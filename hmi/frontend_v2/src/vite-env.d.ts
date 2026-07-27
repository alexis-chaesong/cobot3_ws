/// <reference types="vite/client" />

// import.meta.env 자동완성 및 타입 안전성
interface ImportMetaEnv {
  readonly VITE_MOCK: string;
  readonly VITE_API_BASE: string;
  readonly VITE_WS_URL: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
