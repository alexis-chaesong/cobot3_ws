// 웨이포인트 경로 큐(routeQueue.ts) 구독 훅. MapPanel(경로 표시)과 QueuePanel(큐 목록)이 공유.
import { useEffect, useState } from "react";
import {
  getRouteSnapshot,
  subscribeRouteQueue,
  type RouteMap,
} from "../lib/routeQueue";

export type { RouteMap, RouteWaypoint } from "../lib/routeQueue";

export function useRouteQueue(): RouteMap {
  const [routes, setRoutes] = useState<RouteMap>(getRouteSnapshot);
  useEffect(() => subscribeRouteQueue(setRoutes), []);
  return routes;
}
