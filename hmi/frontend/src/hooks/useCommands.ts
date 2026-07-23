// commands 레이어를 컴포넌트에서 쓰기 편하게 감싼 훅.
// (지금은 그대로 노출하지만, 나중에 "명령 전송 중" 상태 등을 여기 추가하기 좋다.)
import { commands } from "../lib/commands";

export function useCommands() {
  return commands;
}
