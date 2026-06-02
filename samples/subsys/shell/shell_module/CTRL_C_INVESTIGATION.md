# Investigation: interrupting a running shell command with Ctrl+C

**Question:** can we add a Ctrl+C that stops a command while it is running?

**Short answer:** Yes, but only *cooperatively*. A long-running command can be
made interruptible, but a command that does not opt in cannot be stopped safely.
A fully transparent, preemptive Ctrl+C (kill any command mid-execution) is *not*
safely achievable with the current architecture.

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

## Option A — cooperative abort (recommended, demonstrated)

The running command periodically polls the input transport for the ETX (0x03)
byte and returns early when it sees one. Portable across all backends, no ISR or
threading changes, and nothing is killed mid-execution so there are no resource
leaks.

This is demonstrated by `src/ctrlc_poc.c`, which adds a `count` command. Verified
on `native_sim`:

```
uart:~$ count
counting until Ctrl+C ...
tick 0
tick 1
tick 2
tick 3
tick 4
interrupted at 5            <- after Ctrl+C
uart:~$ version             <- shell still responsive
Zephyr version 4.4.99
```

To make this a real feature rather than a per-command trick, the subsystem
would provide a small helper, e.g.:

```c
bool shell_command_interrupted(const struct shell *sh);
```

that does the non-blocking read + 0x03 scan (preserving any non-ETX bytes), so
commands can simply do `if (shell_command_interrupted(sh)) { ... }`. This could
sit behind a `CONFIG_SHELL_CMD_CTRL_C` Kconfig.

**Limitation:** only commands that call the helper can be interrupted. A command
stuck in a tight loop or a blocking call that never polls cannot be stopped.

## Option B — preemptive abort (possible, not recommended)

Run each command in a dedicated worker thread; keep the shell thread reading
input; on 0x03, `k_thread_abort()` the worker.

Drawbacks that make this a poor default:

- `k_thread_abort()` does not unwind. Any mutex held, memory allocated, or
  device left mid-transaction by the command leaks or deadlocks.
- Extra thread + stack (RAM cost) per shell instance.
- Changes execution semantics (command no longer runs in the shell thread);
  shell APIs and `z_flag_cmd_ctx` assumptions would need review.

Feasible for a constrained, opt-in "run this command abortable" wrapper, but not
as transparent behaviour for arbitrary commands.

## Recommendation

Implement Option A as an opt-in helper gated by Kconfig. It is safe, portable,
and matches how interruptible long-running operations are normally written in
Zephyr. Document clearly that interruptibility is cooperative. Treat Option B as
out of scope unless a specific use case justifies the risk.

The `count` command and `src/ctrlc_poc.c` in this sample are a throwaway proof
of concept for the investigation, not a proposed API.
