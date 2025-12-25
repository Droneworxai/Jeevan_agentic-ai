import ROSLIB from 'roslib';

class RosClient {
  private ros: ROSLIB.Ros | null = null;
  private static instance: RosClient;

  private constructor() {}

  public static getInstance(): RosClient {
    if (!RosClient.instance) {
      RosClient.instance = new RosClient();
    }
    return RosClient.instance;
  }

  public connect(url: string = 'ws://localhost:9090'): ROSLIB.Ros {
    if (this.ros && this.ros.isConnected) {
      return this.ros;
    }

    this.ros = new ROSLIB.Ros({
      url: url
    });

    this.ros.on('connection', () => {
      console.log('Connected to ROS bridge.');
    });

    this.ros.on('error', (error) => {
      console.log('Error connecting to ROS bridge: ', error);
    });

    this.ros.on('close', () => {
      console.log('Connection to ROS bridge closed.');
    });

    return this.ros;
  }

  public getRos(): ROSLIB.Ros | null {
    return this.ros;
  }

  public disconnect() {
    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }
  }
}

export default RosClient;
