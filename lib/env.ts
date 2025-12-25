export function validateEnv() {
  const requiredEnvVars = [
    'DATABASE_URL',
    'NEXT_PUBLIC_GEMINI_API_KEY',
  ];

  const missingEnvVars = requiredEnvVars.filter(
    (envVar) => !process.env[envVar]
  );

  if (missingEnvVars.length > 0) {
    throw new Error(
      `The following environment variables are missing: ${missingEnvVars.join(
        ', '
      )}. Please check your .env file.`
    );
  }
}
