"""Set a number (or select) entity by its name, then print the current-related states after `wait` s.
Usage: python set_number.py <host> "<entity name>" <value> [wait_s]   (API key in TWC_API_KEY)
Example: python set_number.py 192.168.1.50 "Set Current" 10"""
import asyncio, os, sys
from aioesphomeapi import APIClient, NumberInfo

WATCH = ("Control Mode", "Max Power", "Assigned Current", "Set Current", "Offered Current", "Actual Current",
         "Charging Power", "Total Power", "State", "Status")


async def main(host, name, value, wait=20):
    cli = APIClient(host, 6053, None, noise_psk=os.environ["TWC_API_KEY"])
    await cli.connect(login=True)
    ents, _ = await cli.list_entities_services()
    names = {e.key: e.name for e in ents}
    ent = next(e for e in ents if e.name == name)
    states = {}
    cli.subscribe_states(lambda s: states.__setitem__(s.key, s))
    if isinstance(ent, NumberInfo):
        cli.number_command(ent.key, float(value))
    else:  # select: value is the option text
        cli.select_command(ent.key, value)
    await asyncio.sleep(float(wait))
    for k, n in names.items():
        if n in WATCH and k in states:
            print(f"{n:18} {states[k].state}")
    await cli.disconnect()


if __name__ == "__main__":
    asyncio.run(main(*sys.argv[1:5]))
