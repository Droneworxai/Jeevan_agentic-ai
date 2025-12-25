Write-Host "Checking for Node.js..."
try {
    node -v | Out-Null
} catch {
    Write-Host "Node.js is not installed. Please install it first."
    exit
}

# Check for .env file
if (-not (Test-Path .env)) {
    Write-Host ".env file not found. Creating from .env.example..."
    if (Test-Path .env.example) {
        Copy-Item .env.example .env
        Write-Host "Please edit .env with your DATABASE_URL and other secrets."
    } else {
        "DATABASE_URL=`"mongodb://localhost:27017/ecoweeder`"" | Out-File -FilePath .env -Encoding utf8
        "NEXT_PUBLIC_GEMINI_API_KEY=`"`"" | Out-File -FilePath .env -Append -Encoding utf8
        Write-Host "Created a basic .env file. Please update it with your actual credentials."
    }
}

Write-Host "Installing dependencies..."
if (Test-Path package-lock.json) {
    npm ci
} else {
    npm install
}

Write-Host "Generating Prisma client..."
npx prisma generate

Write-Host "Starting development server..."
npm run dev