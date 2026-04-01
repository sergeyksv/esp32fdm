#pragma once

/*
 * Host print module — streams GCode from SD card to Marlin printer
 * via USB serial with line numbering, checksums, and resend handling.
 *
 * Internal module header — public API goes through printer_comm.h.
 * See marlin_proto.h for the interface between this and printer_comm.c.
 */
