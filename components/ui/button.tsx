import * as React from "react";
import { cn } from "@/lib/utils";

export interface ButtonProps
  extends React.ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: "default" | "ghost" | "destructive" | "outline";
  size?: "sm" | "md" | "lg" | "icon";
}

const Button = React.forwardRef<HTMLButtonElement, ButtonProps>(
  (
    { className, children, variant = "default", size = "md", ...props },
    ref
  ) => {
    const base =
      "inline-flex items-center justify-center rounded-md text-sm font-medium transition-colors focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring disabled:opacity-50 disabled:pointer-events-none";

    const variantClass =
      variant === "ghost"
        ? "bg-transparent hover:bg-muted"
        : variant === "destructive"
        ? "bg-destructive text-destructive-foreground"
        : variant === "outline"
        ? "border border-input bg-transparent"
        : "bg-primary text-primary-foreground";

    const sizeClass =
      size === "sm"
        ? "h-8 px-2 text-sm"
        : size === "lg"
        ? "h-12 px-6 text-base"
        : size === "icon"
        ? "h-8 w-8 p-0"
        : "h-10 px-4";

    return (
      <button
        ref={ref}
        className={cn(base, variantClass, sizeClass, className)}
        {...props}
      >
        {children}
      </button>
    );
  }
);
Button.displayName = "Button";

export { Button };
