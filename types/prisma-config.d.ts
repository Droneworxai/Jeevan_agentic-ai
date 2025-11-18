declare module "prisma/config" {
  export function defineConfig(config: any): any;
  export const env: (name: string) => string;
}
