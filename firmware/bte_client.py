import asyncio
import curses
import struct
from bleak import BleakScanner, BleakClient

# Configuration
#DEVICE_NAME = "nimble";
DEVICE_NAME = "pwa-86:d3:85:75:dc:3c"
#DEVICE_NAME = "pwa-85:75:dc:3c";
TEMP_UUID = "00002a6e-0000-1000-8000-00805f9b34fb"

async def main(stdscr):
    # Initialize Curses
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()

    # Colors
    curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_MAGENTA, curses.COLOR_BLACK)

    found_devices = {}
    current_temp = "N/A"
    min_temp = None
    max_temp = None
    conn_status = "Disconnected"

    async def scan_callback(device, advertisement_data):
        """Handle incoming advertisements."""
        name = device.name if device.name else "Unknown"
        # Only add if it's not Unknown
        if name != "Unknown":
            found_devices[device.address] = (name, advertisement_data.rssi)

    async def ble_connection_loop():
        """Background task to maintain connection to the ESP32."""
        nonlocal current_temp, min_temp, max_temp, conn_status
        while True:
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=2.0)
            if device:
                try:
                    conn_status = "Connecting..."

                    async with BleakClient(device) as client:
                        conn_status = "Connected"
                        while client.is_connected:
                            # Keep it visible in the list with exact DEVICE_NAME
                            found_devices[device.address] = (DEVICE_NAME, -50)

                            raw = await client.read_gatt_char(TEMP_UUID)
                            val = struct.unpack('f', raw)[0]
                            current_temp = f"{val:.2f} °C"

                            # Update min/max
                            if min_temp is None or val < min_temp:
                                min_temp = val
                            if max_temp is None or val > max_temp:
                                max_temp = val

                            await asyncio.sleep(1)
                except Exception:
                    pass

            conn_status = "Disconnected"
            current_temp = "N/A"
            await asyncio.sleep(2)

    scanner = BleakScanner(
        detection_callback=scan_callback,
        scanning_mode="active"
    )

    await scanner.start()
    asyncio.create_task(ble_connection_loop())

    try:
        while True:
            char = stdscr.getch()
            if char == ord('q'):
                break

            stdscr.clear()
            h, w = stdscr.getmaxyx()

            # Calculate safe column widths based on window size
            available = w - 10
            addr_w = min(36, available - 25)
            rssi_w = 4
            name_w = max(15, available - addr_w - rssi_w - 6)

            # Draw Header
            header = f"{'NAME':<{name_w}} {'ADDRESS':<{addr_w}} {'RSSI':<{rssi_w}}"
            try:
                stdscr.addstr(0, 2, header[:available], curses.A_UNDERLINE | curses.color_pair(1))
            except: pass

            # Sort by RSSI only
            sorted_devices = sorted(found_devices.items(), key=lambda x: x[1][1], reverse=True)

            for i, (addr, (name, rssi)) in enumerate(sorted_devices[:h-6]):
                row_style = curses.color_pair(2) if name == DEVICE_NAME else 0

                display_name = (name[:name_w-2] + '..') if len(name) > name_w else name
                display_addr = addr[:addr_w]

                line = f"{display_name:<{name_w}} {display_addr:<{addr_w}} {rssi:>{rssi_w}} dBm"

                try:
                    stdscr.addstr(i + 1, 2, line[:available], row_style)
                except curses.error:
                    pass

            # Bottom Status Bar
            if h > 4:
                try:
                    stdscr.addstr(h-3, 2, ("─" * (w-4))[:w-4], curses.color_pair(1))
                except: pass

                try:
                    stdscr.addstr(h-2, 2, f"TARGET: {DEVICE_NAME} | STATUS: ", curses.color_pair(3))

                    s_color = curses.color_pair(2) if conn_status == "Connected" else curses.color_pair(4)
                    stdscr.addstr(conn_status, s_color | curses.A_BOLD)

                    stdscr.addstr(" | TEMP: ")
                    stdscr.addstr(current_temp, curses.color_pair(2) | curses.A_BOLD)

                    # Display min/max if available
                    if min_temp is not None and max_temp is not None:
                        stdscr.addstr(f" | MIN: {min_temp:.2f}°C MAX: {max_temp:.2f}°C", curses.color_pair(3))

                    if w > 15:
                        stdscr.addstr(h-2, w-15, "[ 'q' to quit ]", curses.A_DIM)
                except: pass

            stdscr.refresh()
            await asyncio.sleep(2.0)

    finally:
        await scanner.stop()

if __name__ == "__main__":
    try:
        asyncio.run(curses.wrapper(main))
    except KeyboardInterrupt:
        pass
