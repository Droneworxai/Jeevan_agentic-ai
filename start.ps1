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

# Check for ROS (Windows ROS is usually via specific env vars or commands)
if (Get-Command ros2 -ErrorAction SilentlyContinue) {
    $startRos = Read-Host "ROS 2 detected. Would you like to start the ROS Bridge? (y/n)"
    if ($startRos -eq "y") {
        Write-Host "Starting ROS Bridge in background..."
        Start-Process -NoNewWindow -FilePath "ros2" -ArgumentList "launch rosbridge_server rosbridge_websocket_launch.xml"
        
        Write-Host "Starting Mission Planner node..."
        Start-Process -NoNewWindow -FilePath "python" -ArgumentList "simulation/scripts/mission_planner.py"
    }
}

Write-Host "Starting development server..."
npm run dev