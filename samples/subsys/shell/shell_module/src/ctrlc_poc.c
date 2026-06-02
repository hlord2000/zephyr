/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Proof of concept for interrupting a running shell command with Ctrl+C.
 *
 * Background: the shell executes a command handler synchronously in the shell
 * thread. While the handler runs, shell_process()/state_collect() are not
 * called, so the shell itself never looks at incoming bytes. The UART backend
 * is interrupt driven though, so bytes received during command execution are
 * still captured into the backend RX ring buffer.
 *
 * exec_cmd() releases the shell mutex before calling the handler, and the
 * handler runs in the shell thread, so a long running command may itself drain
 * the same non-blocking transport read() that state_collect() normally uses and
 * look for the Ctrl+C (ETX, 0x03) byte. This is the cooperative model: the
 * command voluntarily polls for an abort request. It is portable (no
 * backend/ISR changes) and safe (no thread is killed mid-execution).
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#define ASCII_ETX 0x03 /* Ctrl+C */

/* Returns true if a Ctrl+C byte is waiting on the shell's input transport.
 * Must only be called from within a command handler (i.e. from the shell
 * thread, with the shell mutex released by exec_cmd()).
 */
static bool ctrlc_requested(const struct shell *sh)
{
	uint8_t buf[16];
	size_t count = 0;

	if (sh->iface->api->read(sh->iface, buf, sizeof(buf), &count) < 0) {
		return false;
	}

	for (size_t i = 0; i < count; i++) {
		if (buf[i] == ASCII_ETX) {
			return true;
		}
	}

	return false;
}

static int cmd_count(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "counting until Ctrl+C ...");

	for (int i = 0;; i++) {
		if (ctrlc_requested(sh)) {
			shell_warn(sh, "interrupted at %d", i);
			return -ECANCELED;
		}

		shell_print(sh, "tick %d", i);
		k_sleep(K_MSEC(500));
	}

	return 0;
}

SHELL_CMD_REGISTER(count, NULL, "Count once per 500 ms until Ctrl+C is pressed.",
		   cmd_count);
