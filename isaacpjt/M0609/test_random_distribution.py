"""
_randomize_active_cube() 안의 random.choice(["blue", "green"]) 로직만 떼어내서
Isaac Sim 없이 순수 파이썬으로 분포를 확인하는 테스트.

목적: "계속 파란색만 나온다"는 증상이 (a) random 모듈/시드 자체의 문제인지,
      (b) Isaac Sim 쪽 시각화/물리 반영 로직의 문제인지 구분하기 위함.

판단 기준:
  - 이 스크립트에서도 blue/green이 편향되게 나온다 → random 모듈/시드 문제.
  - 이 스크립트는 대략 50:50에 가깝게 나온다 → 문제는 Isaac Sim 쪽
    (visibility/enable_rigid_body_physics 반영, world.reset() 타이밍 등)에 있다.

사용법:
    python3 test_random_distribution.py            # 기본 50회
    python3 test_random_distribution.py 200         # 200회로 변경
"""

import random
import sys
from collections import Counter

N = int(sys.argv[1]) if len(sys.argv) > 1 else 50


def main():
    # 6_pick_place_color.py 의 _randomize_active_cube() 와 완전히 동일한 호출 형태.
    # random.seed()는 의도적으로 호출하지 않는다 (실제 코드에도 시드 고정이 없으므로
    # 동일한 조건에서 테스트하기 위함).
    results = []
    for i in range(N):
        choice = random.choice(["blue", "green"])
        results.append(choice)
        print(f"  [{i + 1:3d}] random.choice 결과 = {choice}")

    counts = Counter(results)
    blue_n = counts["blue"]
    green_n = counts["green"]

    print("\n" + "=" * 40)
    print(f"총 {N}회 중 blue: {blue_n}번, green: {green_n}번")
    print(f"blue 비율: {blue_n / N * 100:.1f}%  /  green 비율: {green_n / N * 100:.1f}%")
    print("=" * 40)

    if blue_n == 0 or green_n == 0:
        print("[결론] 한쪽이 0번 → random 모듈/시드 자체에 심각한 문제가 있음.")
    elif abs(blue_n - green_n) / N > 0.3:
        print("[결론] 30% 이상 치우침 → random 모듈/시드 쪽을 의심할 것.")
    else:
        print("[결론] 대체로 균등 → random.choice 자체는 정상. "
              "문제는 Isaac Sim 쪽(visibility/physics 반영, reset 타이밍)에 있을 가능성이 높음.")


if __name__ == "__main__":
    main()
