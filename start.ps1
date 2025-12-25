Write-Host "Checking for Node.js..."
try {
    node -v | Out-Null
} catch {
    Write-Host "Node.js is not installed. Please install it first."
    exit
}

Write-Host "Installing dependencies..."
npm install

Write-Host "Generating Prisma client..."
npx prisma generate

Write-Host "Starting development server..."
npm run dev