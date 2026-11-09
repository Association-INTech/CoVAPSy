from typing import BinaryIO


class FifoClosedError(EOFError):
    pass


def read_exact(pipe: BinaryIO, size: int, context: str) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = pipe.read(size - len(data))
        if not chunk:
            raise FifoClosedError(
                f"{context}: FIFO closed after {len(data)} of {size} bytes"
            )
        data.extend(chunk)
    return bytes(data)
