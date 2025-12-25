"use client";

import React, { createElement, Fragment, useEffect, useRef } from "react";
import { createRoot } from "react-dom/client";

import { autocomplete } from "@algolia/autocomplete-js";
import type { AutocompleteOptions } from "@algolia/autocomplete-js";
import type { BaseItem } from "@algolia/autocomplete-core";

import "@algolia/autocomplete-theme-classic";
import "./Search.css";

type AutocompleteProps = Partial<AutocompleteOptions<BaseItem>> & {
  className?: string;
};

export function Autocomplete(props: AutocompleteProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const roots = useRef(new Map<HTMLElement, any>());

  useEffect(() => {
    if (!containerRef.current) {
      return undefined;
    }

    const search = autocomplete({
      container: containerRef.current,
      renderer: {
        createElement,
        Fragment,
        render(children, container) {
          let root = roots.current.get(container);
          if (!root) {
            root = createRoot(container);
            roots.current.set(container, root);
          }
          root.render(children);
        },
      },
      ...props,
    });

    return () => {
      search.destroy();
      roots.current.forEach((root) => root.unmount());
      roots.current.clear();
    };
  }, [props]);

  return <div className={props.className} ref={containerRef} />;
}
