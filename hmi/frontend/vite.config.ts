import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    host: true, // 관제실 PC 외부(같은 네트워크)에서 접속할 필요가 있으면 유지
  },
});
