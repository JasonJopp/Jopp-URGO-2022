import asyncio
from drive import DriveHandler

driver = DriveHandler()

async def main():
    await asyncio.run(driver.drive('forward'))

asyncio.run(main())
