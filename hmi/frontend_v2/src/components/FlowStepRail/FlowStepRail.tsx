// steps 배열 + current 인덱스를 받아 단계 레일을 그린다. 색은 variant로 분기.
import { Check } from "lucide-react";
import "./FlowStepRail.css";

interface Props {
  steps: readonly string[];
  currentStep: number; // 강조할 현재 단계 인덱스
  variant: "waste" | "disinfect";
}

export function FlowStepRail({ steps, currentStep, variant }: Props) {
  return (
    <ol className={`flow-rail flow-rail--${variant}`}>
      {steps.map((label, i) => {
        const status =
          i < currentStep ? "done" : i === currentStep ? "current" : "todo";
        return (
          <li
            key={`${i}-${label}`}
            className={`flow-step flow-step--${status}`}
            title={label}
          >
            <span className="flow-step__dot">
              {status === "done" ? <Check size={12} strokeWidth={3} /> : i + 1}
            </span>
            <span className="flow-step__label">{label}</span>
          </li>
        );
      })}
    </ol>
  );
}
