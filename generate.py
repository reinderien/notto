#!/usr/bin/env python3
import random
from pathlib import Path


def generate_malformed_huge_random(dest: Path, n: int) -> None:
    """
    :param n: Arbitrarily large, though this can technically produce malformed
    output since there is no effort made to keep the waypoints unique
    """
    with dest.open('w') as f:
        print(n, file=f)
        for _ in range(n):
            print(
                *(random.randrange(1, 100) for _ in range(3)),
                file=f,
            )
        print(0, file=f)


def generate_max_random(dest: Path) -> None:
    all_coords = [
        (x, y)
        for x in range(1, 100)
        for y in range(1, 100)
    ]
    random.shuffle(all_coords)

    with dest.open('w') as f:
        print(len(all_coords), file=f)
        for x, y in all_coords:
            print(
                x, y, random.randrange(1, 100),
                file=f,
            )
        print(0, file=f)


def generate_lowdist(dest: Path, penalty: int) -> None:
    all_triples = [
        (x, 99, penalty)
        for x in range(1, 100)
    ]

    with dest.open('w') as f:
        print(len(all_triples), file=f)
        for triple in all_triples:
            print(*triple, file=f)
        print(0, file=f)


if __name__ == '__main__':
    parent = Path('generated/')
    parent.mkdir(exist_ok=True)
    generate_lowdist(parent / 'lowdist_lowpen.txt', penalty=1)
    generate_lowdist(parent / 'lowdist_hipen.txt', penalty=100)
    generate_max_random(parent / 'max.txt')
    generate_malformed_huge_random(parent / 'big.txt', n=1_000_000)
