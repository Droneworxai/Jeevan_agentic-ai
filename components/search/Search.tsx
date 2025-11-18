"use client";

import React, { createElement, Fragment, useEffect, useRef } from "react";
import { createRoot } from "react-dom/client";

import { autocomplete } from "@algolia/autocomplete-js";
import type { AutocompleteOptions } from "@algolia/autocomplete-js";
import type { BaseItem } from "@algolia/autocomplete-core";

import "@algolia/autocomplete-theme-classic";

type AutocompleteProps = Partial<AutocompleteOptions<BaseItem>> & {
  className?: string;
};

export function Autocomplete(props: AutocompleteProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const rootRef = useRef<any>(null);

  useEffect(() => {
    if (!containerRef.current) {
      return undefined;
    }

    if (!rootRef.current) {
      rootRef.current = createRoot(containerRef.current);
    }

    const search = autocomplete({
      container: containerRef.current,
      renderer: { createElement, Fragment, render: rootRef.current.render },
      ...props,
    });

    return () => {
      search.destroy();
    };
  }, [props]);

  return <div className={props.className} ref={containerRef} />;
}
