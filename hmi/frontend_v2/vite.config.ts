import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// https://vitejs.dev/config/
// [HMI v2] 기존 hmi/frontend(5173)와 병행 실행 — 포트만 5174로 분리.
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5174,
    host: true, // 관제실 PC 외부(같은 네트워크)에서 접속할 필요가 있으면 유지
  },
});
