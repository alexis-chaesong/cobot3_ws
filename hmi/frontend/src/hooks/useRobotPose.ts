// carter1/carter2 실시간 위치 표시용 훅. 실제 구독 로직은 lib/robotPoseStore.ts(모듈 싱글턴)에
// 있고, 여기서는 그 스토어를 구독만 한다(routeQueue.ts 도 같은 스토어를 공유).
import { useEffect, useState } from "react";
import {
  getPoseSnapshot,
  subscribeRobotPoseStore,
  type PoseMap,
} from "../lib/robotPoseStore";

export type { PoseMap } from "../lib/robotPoseStore";

export function useRobotPose(): PoseMap {
  const [poses, setPoses] = useState<PoseMap>(getPoseSnapshot);
  useEffect(() => subscribeRobotPoseStore(setPoses), []);
  return poses;
}
