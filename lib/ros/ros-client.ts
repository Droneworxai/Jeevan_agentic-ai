import * as ROSLIB from 'roslib';

class RosClient {
  private ros: ROSLIB.Ros | null = null;
  private static instance: RosClient;
  private url: string = 'ws://localhost:9090';
  private reconnectTimeout: any = null;
  private isConnecting: boolean = false;

  private constructor() {}

  public static getInstance(): RosClient {
    if (!RosClient.instance) {
      RosClient.instance = new RosClient();
    }
    return RosClient.instance;
  }

  public connect(url: string = 'ws://localhost:9090'): ROSLIB.Ros {
    this.url = url;
    
    if (this.ros && (this.ros.isConnected || this.isConnecting)) {
      return this.ros;
    }

    this.isConnecting = true;
    this.ros = new ROSLIB.Ros({
      url: this.url
    });

    this.ros.on('connection', () => {
      console.log('Connected to ROS bridge.');
      this.isConnecting = false;
      if (this.reconnectTimeout) {
        clearTimeout(this.reconnectTimeout);
        this.reconnectTimeout = null;
      }
    });

    this.ros.on('error', (error) => {
      console.log('Error connecting to ROS bridge: ', error);
      this.isConnecting = false;
      this.scheduleReconnect();
    });

    this.ros.on('close', () => {
      console.log('Connection to ROS bridge closed.');
      this.isConnecting = false;
      this.scheduleReconnect();
    });

    return this.ros;
  }

  private scheduleReconnect() {
    if (this.reconnectTimeout) return;
    
    console.log('Scheduling reconnect in 5 seconds...');
    this.reconnectTimeout = setTimeout(() => {
      this.reconnectTimeout = null;
      this.connect(this.url);
    }, 5000);
  }

  public getRos(): ROSLIB.Ros | null {
    return this.ros;
  }

  public disconnect() {
    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout);
      this.reconnectTimeout = null;
    }
    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }
    this.isConnecting = false;
  }
}

export default RosClient;
