import dispatcher from '../dispatcher';

import RosLib from 'roslib';

class CommonActions {
    constructor() {
        this.createRosClient('localhost', '9090');
    }

    createRosClient = (hostname, port) => {
        const rosUrl = 'ws://' + hostname + ':' + port;

        let rosClient = new RosLib.Ros();

        rosClient.on('connection', () => {
            console.log('Connected to websocket server.');
            dispatcher.dispatch({
                type: 'ROS_CONNECTION_STATUS',
                rosConnectionStatus: 'connected'
            });
            this.rosClient = rosClient;
            this.connectRos();
        });

        rosClient.on('error', (error) => {
            console.log('Error connecting to websocket server: ', error);

            dispatcher.dispatch({
                type: 'ROS_CONNECTION_STATUS',
                rosConnectionStatus: 'error'
            });
        });

        rosClient.on('close', () => {
            console.log('Disconnected from websocket server.');
            dispatcher.dispatch({
                type: 'ROS_CONNECTION_STATUS',
                rosConnectionStatus: 'disconnected'
            });
        });

        rosClient.connect(rosUrl);

        dispatcher.dispatch({
            type: 'ROS_CLIENT',
            rosClient: rosClient
        });
    }

    connectRos = () =>
    {
        console.log("Connecting to topics!");
        this.namespace = '';

        const nodeName = 'canopen_chain';

        this.changeLifecycleChaneStateService = new RosLib.Service({
            ros: this.rosClient,
            name: '/canopen_chain/change_state',
            serviceType: 'lifecycle_msgs/srv/ChangeState'
        });
    }

    callLifecycleChangeStateService = (transition) =>
    {
        const request = new RosLib.ServiceRequest({
            transition: {
                id: '',
                label: transition
            }
        });

        console.log("transition clicked");
        console.log(this.changeLifecycleChaneStateService);

        this.changeLifecycleChaneStateService.callService(request, result => {
            console.log("Yey, transition success!!");
        })
    }

}

const commonActions = new CommonActions();

export default commonActions;