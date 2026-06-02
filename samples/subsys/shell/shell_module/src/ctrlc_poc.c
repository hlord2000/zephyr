/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Proof of concept for interrupting a running shell command with Ctrl+C using
 * the preemptive CONFIG_SHELL_CMD_ABORT mechanism.
 *
 * With CONFIG_SHELL_CMD_ABORT enabled, interactive command handlers run in a
 * worker thread while the shell thread keeps watching the input. A command opts
 * in to being interruptible with a single call:
 *
 *     shell_command_set_abort_handler(sh, cb, ctx);
 *
 * When Ctrl+C is received, the shell thread aborts the worker and calls cb()
 * (from the shell thread) so the command can release its resources. The command
 * loop itself needs no checkpoints - the abort is preemptive.
 *
 * To keep the demo self-contained the "resource" here is a heap allocation; the
 * abort handler frees it. Note the callback must only release things that are
 * safe to touch cross-thread (heap, k_sem, device cancel) - not a k_mutex held
 * by the command.
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

static void count_abort(const struct shell *sh, void *user_data)
{
	/* Runs in the shell thread after the worker has been aborted. */
	k_free(user_data);
	shell_warn(sh, "interrupted; resources released");
}

static int cmd_count(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* A resource the command owns and that the abort handler must release. */
	void *resource = k_malloc(64);

	shell_command_set_abort_handler(sh, count_abort, resource);

	shell_print(sh, "counting until Ctrl+C ...");

	for (int i = 0;; i++) {
		shell_print(sh, "tick %d", i);
		k_sleep(K_MSEC(500));
	}

	/* Not reached, but a well-behaved command would free here on the normal
	 * exit path and clear the abort handler.
	 */
	k_free(resource);
	return 0;
}

SHELL_CMD_REGISTER(count, NULL, "Count once per 500 ms until Ctrl+C is pressed.",
		   cmd_count);
