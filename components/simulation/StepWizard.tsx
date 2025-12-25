"use client";

import { Bot } from "lucide-react";
import { Button } from "@/components/ui/button";

interface Step {
  title: string;
  description: string;
  note: string;
  action: (() => void) | null;
  canComplete: () => boolean;
}

interface StepWizardProps {
  currentStep: number;
  setCurrentStep: (step: number) => void;
  completedSteps: number[];
  setCompletedSteps: (steps: number[]) => void;
  showInstructions: boolean;
  setShowInstructions: (show: boolean) => void;
  steps: Step[];
  isLoading: boolean;
}

export default function StepWizard({
  currentStep,
  setCurrentStep,
  completedSteps,
  setCompletedSteps,
  showInstructions,
  setShowInstructions,
  steps,
  isLoading,
}: StepWizardProps) {
  return (
    <div 
      className="bg-white border border-gray-300 rounded-lg shadow-sm sticky top-4 z-50 cursor-pointer hover:border-blue-400 transition-colors"
      onClick={() => setShowInstructions(!showInstructions)}
    >
      <div className="px-3 py-2 flex items-center justify-between">
        <div className="flex items-center gap-2">
          <Bot className="w-4 h-4 text-blue-600" />
          <span className="text-xs font-bold text-gray-800">
            Step {currentStep + 1}/{steps.length}: {steps[currentStep].title}
          </span>
          {/* Mini Progress Dots */}
          <div className="flex gap-1 ml-2">
            {steps.map((_, idx) => (
              <div 
                key={idx}
                className={`w-1.5 h-1.5 rounded-full ${
                  completedSteps.includes(idx) ? 'bg-green-500' : 
                  currentStep === idx ? 'bg-blue-500' : 
                  'bg-gray-300'
                }`}
              />
            ))}
          </div>
        </div>
        <span className="text-xs text-gray-500">{showInstructions ? '▼' : '▶'}</span>
      </div>
      
      {showInstructions && (
        <div className="px-3 py-2 border-t border-gray-200 bg-gray-50">
          <div className="text-xs space-y-2">
            <p className="text-gray-700"><strong>{steps[currentStep].description}</strong></p>
            <p className="text-gray-600 text-[11px]">💡 {steps[currentStep].note}</p>
            
            {/* Compact Action Buttons */}
            <div className="flex items-center justify-between pt-2">
              <Button
                variant="outline"
                size="sm"
                onClick={(e) => {
                  e.stopPropagation();
                  setCurrentStep(Math.max(0, currentStep - 1));
                }}
                disabled={currentStep === 0}
                className="h-7 text-xs px-2"
              >
                ← Prev
              </Button>

              <div className="flex gap-2">
                {steps[currentStep].action && (
                  <Button
                    size="sm"
                    onClick={(e) => {
                      e.stopPropagation();
                      steps[currentStep].action!();
                    }}
                    disabled={!steps[currentStep].canComplete() || isLoading}
                    className="h-7 text-xs px-2 bg-green-600 hover:bg-green-700"
                  >
                    {isLoading ? "..." : "Do It"}
                  </Button>
                )}
                
                <Button
                  size="sm"
                  onClick={(e) => {
                    e.stopPropagation();
                    if (steps[currentStep].canComplete()) {
                      if (!completedSteps.includes(currentStep)) {
                        setCompletedSteps([...completedSteps, currentStep]);
                      }
                      setCurrentStep(Math.min(steps.length - 1, currentStep + 1));
                    }
                  }}
                  disabled={!steps[currentStep].canComplete() || currentStep === steps.length - 1}
                  className="h-7 text-xs px-2 bg-blue-600 hover:bg-blue-700"
                >
                  {currentStep === steps.length - 1 ? "✓" : "Next →"}
                </Button>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
