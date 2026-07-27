// carterId 하나의 스냅샷을 꺼내는 얇은 훅.
import { useRobotStatusContext } from "../context/RobotStatusContext";
import type { CarterId, RobotSnapshot } from "../types";

export function useRobotStatus(carterId: CarterId): RobotSnapshot {
  const { snapshots } = useRobotStatusContext();
  return snapshots[carterId];
}
