"""TOFSense-M UART input to one-way MAVLink range output."""
import os
import socket
import termios
import time
import tty

from .tofsense import (TofHeightKalman, TofSenseMDecoder, distance_sensor_v1,
                       tof_ipc_packet)


def configure(fd, baud):
    tty.setraw(fd)
    cfg = termios.tcgetattr(fd)
    cfg[4] = cfg[5] = baud
    cfg[2] |= termios.CLOCAL | termios.CREAD
    cfg[2] &= ~termios.CRTSCTS
    termios.tcsetattr(fd, termios.TCSANOW, cfg)


def main():
    filter_mode = os.environ.get('TOF_HEIGHT_FILTER', 'raw').lower()
    if filter_mode not in ('raw', 'kalman'):
        raise ValueError('TOF_HEIGHT_FILTER must be raw or kalman')
    height_filter = TofHeightKalman() if filter_mode == 'kalman' else None
    source = os.open('/dev/ttyS3', os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    output = os.open('/dev/ttyS1', os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
    ipc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    source_saved, output_saved = termios.tcgetattr(source), termios.tcgetattr(output)
    configure(source, termios.B921600)
    configure(output, termios.B57600)
    termios.tcflush(source, termios.TCIFLUSH)
    decoder, sequence, started = TofSenseMDecoder(), 0, time.monotonic()
    last_report_ns = time.monotonic_ns()
    last_data_read_ns = last_forward_ns = None
    last_forward_counts = (0, 0, 0, 0)
    read_bytes = read_calls = forwarded = batch_drops = filter_drops = 0
    uart_write_blocked = ipc_errors = 0

    def report_if_due(now_ns):
        nonlocal last_report_ns
        if now_ns - last_report_ns >= 10_000_000_000:
            age_ms = ((now_ns - last_forward_ns) / 1e6
                      if last_forward_ns is not None else float('inf'))
            print('TOF_DIAG '
                  f'read_calls={read_calls} read_bytes={read_bytes} '
                  f'frames_seen={decoder.frames_seen} valid={decoder.frames_valid} '
                  f'crc_errors={decoder.crc_errors} mode_rejected={decoder.mode_rejected} '
                  f'range_rejected={decoder.range_rejected} forwarded={forwarded} '
                  f'batch_drops={batch_drops} filter_mode={filter_mode} '
                  f'filter_drops={filter_drops} uart_write_blocked={uart_write_blocked} '
                  f'ipc_errors={ipc_errors} last_forward_age_ms={age_ms:.1f}',
                  flush=True)
            last_report_ns = now_ns

    try:
        while True:
            try:
                data = os.read(source, 4096)
            except BlockingIOError:
                report_if_due(time.monotonic_ns())
                time.sleep(.002)
                continue
            read_ns = time.monotonic_ns()
            if data:
                read_calls += 1
                read_bytes += len(data)
                if last_data_read_ns is not None:
                    read_gap_ms = (read_ns - last_data_read_ns) / 1e6
                    if read_gap_ms > 100:
                        print(f'TOF_RAW_READ_GAP gap_ms={read_gap_ms:.1f} '
                              f'bytes={len(data)} frames_seen={decoder.frames_seen} '
                              f'valid={decoder.frames_valid}', flush=True)
                last_data_read_ns = read_ns
            rows = decoder.feed(data)
            if len(rows) > 1:  # never publish queued measurements as fresh
                batch_drops += len(rows)
                if height_filter is not None:
                    height_filter.reset()
                print(f'TOF_BATCH_DROP count={len(rows)} '
                      f'frames_seen={decoder.frames_seen} valid={decoder.frames_valid}',
                      flush=True)
                report_if_due(read_ns)
                continue
            for unused_boot, distance in rows:
                receipt_ns = time.monotonic_ns()
                counts = (decoder.frames_seen, decoder.crc_errors,
                          decoder.mode_rejected, decoder.range_rejected)
                if last_forward_ns is not None:
                    gap_ms = (receipt_ns - last_forward_ns) / 1e6
                    if gap_ms > 100:
                        delta = tuple(current - previous for current, previous
                                      in zip(counts, last_forward_counts))
                        print(f'TOF_VALID_GAP gap_ms={gap_ms:.1f} '
                              f'frames_seen_delta={delta[0]} crc_delta={delta[1]} '
                              f'mode_delta={delta[2]} range_delta={delta[3]} '
                              f'device_ticks={unused_boot:.3f} distance_m={distance:.3f}',
                              flush=True)
                if height_filter is not None:
                    filtered = height_filter.update(receipt_ns / 1e9, distance)
                    if filtered is None:
                        filter_drops += 1
                        continue
                    distance = filtered
                last_forward_ns, last_forward_counts = receipt_ns, counts
                packet = distance_sensor_v1(
                    distance, sequence, int((time.monotonic()-started)*1000))
                try:
                    os.write(output, packet)
                except BlockingIOError:
                    uart_write_blocked += 1
                    continue
                sequence = (sequence + 1) & 255
                forwarded += 1
                # Height delivery to the flight controller is primary.  A
                # local preview consumer must never be able to terminate it.
                try:
                    ipc.sendto(tof_ipc_packet(receipt_ns, unused_boot, distance),
                               ('127.0.0.1', 14581))
                except OSError:
                    ipc_errors += 1
            report_if_due(time.monotonic_ns())
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(source, termios.TCSANOW, source_saved)
        termios.tcsetattr(output, termios.TCSANOW, output_saved)
        os.close(source)
        os.close(output)
        ipc.close()


if __name__ == '__main__':
    main()
