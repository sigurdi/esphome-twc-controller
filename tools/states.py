"""Print all entity states of a device.
Usage: python states.py <host> [api-key]   (or set TWC_API_KEY; the key is api: encryption: key in your YAML)"""
import asyncio, os, sys
from aioesphomeapi import APIClient


async def main(host, key):
    cli = APIClient(host, 6053, None, noise_psk=key)
    await cli.connect(login=True)
    ents, _ = await cli.list_entities_services()
    names = {e.key: e.name for e in ents}
    states = {}
    cli.subscribe_states(lambda s: states.__setitem__(s.key, s))
    await asyncio.sleep(3)
    for k, n in sorted(names.items(), key=lambda x: x[1]):
        s = states.get(k)
        v = getattr(s, "state", None) if s else None
        if s is not None and getattr(s, "missing_state", False):
            v = "unknown"
        print(f"{n:24} {v}")
    await cli.disconnect()


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else os.environ["TWC_API_KEY"]))
