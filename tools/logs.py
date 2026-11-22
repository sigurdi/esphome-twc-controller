"""Print device log lines containing a text, for N seconds.
Usage: python logs.py <host> <seconds> [filter]   (API key in TWC_API_KEY)
Example: python logs.py 192.168.1.50 60 "L1 check" """
import asyncio, os, re, sys
from aioesphomeapi import APIClient, LogLevel


async def main(host, secs, flt=""):
    cli = APIClient(host, 6053, None, noise_psk=os.environ["TWC_API_KEY"])
    await cli.connect(login=True)

    def on(msg):
        line = re.sub(r"\x1b\[[0-9;]*m", "", msg.message.decode(errors="replace"))
        if flt.lower() in line.lower():
            print(line, flush=True)

    cli.subscribe_logs(on, log_level=LogLevel.LOG_LEVEL_DEBUG)
    await asyncio.sleep(float(secs))
    await cli.disconnect()


if __name__ == "__main__":
    asyncio.run(main(*sys.argv[1:4]))
