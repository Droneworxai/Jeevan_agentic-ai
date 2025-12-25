#!/bin/bash

# IMPORTANT in Linux: Run chmod +x start.sh then ./start.sh

echo "Checking for Node.js..."
node -v > /dev/null 2>&1 || { echo "Node.js is not installed. Please install it first."; exit 1; }

# Check for .env file
if [ ! -f .env ]; then
    echo ".env file not found. Creating from .env.example..."
    if [ -f .env.example ]; then
        cp .env.example .env
        echo "Please edit .env with your DATABASE_URL and other secrets."
    else
        echo "DATABASE_URL=\"mongodb://localhost:27017/ecoweeder\"" > .env
        echo "NEXT_PUBLIC_GEMINI_API_KEY=\"\"" >> .env
        echo "Created a basic .env file. Please update it with your actual credentials."
    fi
fi

echo "Installing dependencies..."
if [ -f package-lock.json ]; then
    npm ci
else
    npm install
fi

echo "Generating Prisma client..."
npx prisma generate

echo "Starting development server..."
npm run dev