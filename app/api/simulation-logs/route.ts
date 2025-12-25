import { NextResponse } from "next/server";
import fs from "fs";
import path from "path";

export async function GET() {
  try {
    const logFilePath = path.join(process.cwd(), "simulation.log");
    
    // Check if log file exists
    if (!fs.existsSync(logFilePath)) {
      return NextResponse.json({ logs: "" }, { status: 200 });
    }

    // Read the last 50KB of the file for performance
    const stats = fs.statSync(logFilePath);
    const fileSize = stats.size;
    const readSize = Math.min(fileSize, 50000); // 50KB
    
    const buffer = Buffer.alloc(readSize);
    const fd = fs.openSync(logFilePath, "r");
    fs.readSync(fd, buffer, 0, readSize, Math.max(0, fileSize - readSize));
    fs.closeSync(fd);
    
    const logs = buffer.toString("utf-8");
    
    return NextResponse.json({ logs }, { status: 200 });
  } catch (error) {
    console.error("Error reading simulation logs:", error);
    return NextResponse.json(
      { error: "Failed to read simulation logs", logs: "" },
      { status: 500 }
    );
  }
}
