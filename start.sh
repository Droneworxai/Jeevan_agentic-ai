# IMPORTANT in Linux: Run chmod +x start.sh then ./start.sh

#!/bin/bash
echo "Checking for Node.js..."
node -v > /dev/null 2>&1 || { echo "Node.js is not installed. Please install it first."; exit 1; }

echo "Installing dependencies..."
npm install

echo "Generating Prisma client..."
npx prisma generate

echo "Starting development server..."
npm run dev