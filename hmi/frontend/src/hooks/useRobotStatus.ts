// robotId 하나의 스냅샷을 꺼내는 얇은 훅.
import { useRobotStatusContext } from "../context/RobotStatusContext";
import type { RobotId, RobotSnapshot } from "../types";

export function useRobotStatus(robotId: RobotId): RobotSnapshot {
  const { snapshots } = useRobotStatusContext();
  return snapshots[robotId];
}
