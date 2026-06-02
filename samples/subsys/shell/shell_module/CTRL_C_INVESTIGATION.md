# Investigation: interrupting a running shell command with Ctrl+C

**Question:** can we add a Ctrl+C that stops a command while it is running?

**Short answer:** Yes, for commands that opt in. A command cannot be stopped
unless it opts in, because the shell runs handlers synchronously in its own
thread. Two opt-in models work: cooperative polling (Option A) and preemptive
abort via a worker thread (Option B). This branch prototypes **Option B** behind
`CONFIG_SHELL_CMD_ABORT` — a command becomes interruptible by adding one line.
A truly *transparent* Ctrl+C for arbitrary, unmodified commands remains unsafe
(aborting code that holds a `k_mutex` or owns un-released resources).

## Why naive Ctrl+C does not work

The shell is single-threaded and runs command handlers synchronously:

```
shell_thread()                     subsys/shell/shell.c:1816
  k_event_wait(RXRDY|LOG_MSG|KILL)
  shell_process()  -> state_collect()  -> execute() -> exec_cmd()
                                                          handler(sh, argc, argv)   <-- blocks here
```

While `handler()` runs, the shell thread is *inside* `exec_cmd()` (the handler
call is at `subsys/shell/shell.c:983`) and never returns to `k_event_wait()` /
`state_collect()`. So the shell never reads or interprets input, and a Ctrl+C
byte is not looked at until the command has already finished.

The existing Ctrl+C handling (`SHELL_VT100_ASCII_CTRL_C` in
`ctrl_metakeys_handle()`, `subsys/shell/shell.c:1361`) only runs during *line
editing* — it abandons the line currently being typed. It has no effect during
command execution.

## Key enabling fact

The UART backend RX is interrupt driven (`uart_rx_handle()` /`async_callback()`
in `subsys/shell/backends/shell_uart.c`). Bytes received *while a command runs*
are still captured into the backend RX ring buffer by the ISR — they are simply
never inspected. So the data is available; what is missing is something to look
at it during execution.

Two more facts make a cooperative solution clean:

1. `exec_cmd()` releases the shell mutex around the handler call
   (`z_shell_unlock()` at `subsys/shell/shell.c:976`, re-locked at `:986`,
   handler call at `:983`).
2. The handler runs in the shell thread, which is the only consumer of the
   transport `read()`. So a handler may drain the same non-blocking
   `sh->iface->api->read()` that `state_collect()` uses, with no contention.

## Option A — cooperative abort

The running command periodically polls the input transport for the ETX (0x03)
byte and returns early when it sees one. Portable across all backends, no ISR or
threading changes, and nothing is killed mid-execution so there are no resource
leaks. A subsystem helper such as `bool shell_command_interrupted(const struct
shell *sh)` (non-blocking read + 0x03 scan) would let commands write
`if (shell_command_interrupted(sh)) { ... cleanup; return; }`.

**Limitation:** only commands that poll can be interrupted, and the command
carries the checkpoint boilerplate.

## Option B — preemptive abort (prototyped here)

Run the command in a worker thread; keep the shell thread reading input; on
Ctrl+C, `k_thread_abort()` the worker and run a registered cleanup handler. This
is the model implemented on this branch (see below). The cost is an extra worker
thread/stack and the cleanup constraints described later; the benefit is that a
command becomes interruptible by adding a single line, with no checkpoints.

## What is prototyped on this branch (Option B)

This branch implements the preemptive model behind `CONFIG_SHELL_CMD_ABORT`
(off by default), optimised for a minimal command-side change. A command opts in
with a single call:

```c
static void my_abort(const struct shell *sh, void *ctx)
{
    k_free(ctx);                 /* release resources (cross-thread safe only) */
}

static int cmd_long(const struct shell *sh, size_t argc, char **argv)
{
    void *res = k_malloc(...);
    shell_command_set_abort_handler(sh, my_abort, res);   /* the only added line */

    for (;;) { do_work(); k_sleep(...); }   /* no checkpoints needed */
}
```

### How it works

- When `CONFIG_SHELL_CMD_ABORT=y`, `exec_cmd()` runs interactive handlers
  (those entered on the shell thread, `k_current_get() == sh->ctx->tid`) in a
  shared worker thread, created one priority level **below** the shell thread so
  the shell thread always preempts to poll input. `shell_execute_cmd()` called
  from other threads still runs inline.
- The shell thread sits in a poll loop (`exec_cmd_abortable()`): it waits on a
  completion semaphore with a `CONFIG_SHELL_CMD_ABORT_POLL_MS` timeout and, each
  tick, does a non-blocking transport read looking for ETX (0x03).
- On Ctrl+C, if a cleanup handler is registered, the shell thread takes the
  shell lock (so any in-flight `shell_print()` in the worker finishes and
  releases it — guaranteeing the worker does not hold it at abort), aborts the
  worker, releases the lock, then calls the cleanup handler and returns
  `-ECANCELED`.

The shell "lock" is a counting `k_sem` (`lock_sem`), not a `k_mutex`, so the
shell thread can manage it across threads safely. The cleanup handler is invoked
**with the lock released**, so it may itself call `shell_print()`.

### Verified on native_sim

```
uart:~$ count
counting until Ctrl+C ...
tick 0 ... tick 4
interrupted; resources released   <- cleanup handler ran (heap freed)
uart:~$ help                      <- shell still responsive
```

Normal commands (`version`, `help`, ...) still work unchanged through the worker
path, and the existing shell ztest suite passes with the option off.

### Limitations (important)

- **Cleanup cannot release a `k_mutex` held by the command.** The handler runs
  in the shell thread, and `k_mutex_unlock()` is owner-only (returns `-EPERM`,
  kernel/mutex.c:246); `k_thread_abort()` also does not release mutexes the
  worker held. Interruptible commands must not hold a `k_mutex` across the
  interruptible region — use a `k_sem`/heap/device-cancel that the handler can
  release cross-thread.
- **Pure CPU-bound commands cannot be interrupted under native_sim.** native_sim
  only advances simulated time when threads yield to the kernel, so a worker that
  never makes a kernel call freezes the clock and the shell thread's poll timeout
  never fires. On real hardware the system-timer ISR fires in real time and the
  higher-priority shell thread preempts, so this case works there. A command in a
  tight loop should still call `k_yield()`/`k_sleep()` occasionally.
- **One interruptible command at a time** across all shell instances (single
  shared worker thread/stack), and extra RAM for that stack.
- Bytes typed while a command runs are consumed by the poll loop and discarded.

The `count` command / `src/ctrlc_poc.c` are a throwaway demonstration, not a
proposed API surface.
