#!/usr/bin/env python3
import random
from pathlib import Path


def generate(dest: Path, n: int) -> None:
    with dest.open('w') as f:
        print(n, file=f)
        for _ in range(n):
            print(
                *(random.randrange(1, 100) for _ in range(3)),
                file=f,
            )
        print(0, file=f)


generate(Path('big.txt'), 25_000)
