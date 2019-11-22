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
            console.log('Connected to ros-web-bridge server.');
            dispatcher.dispatch({
                type: 'ROS_CONNECTION_STATUS',
                rosConnectionStatus: 'connected'
            });
            this.rosClient = rosClient;
            this.connectRos();
            this.refreshState();
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

    connectRos = () => {
        this.namespace = '';
        this.nodeName = 'canopen_chain'

        this.createServiceCallers();
        this.createSubscriptions();
    }

    createServiceCallers = () => 
    {
        this.changeLifecycleChaneStateService = new RosLib.Service({
            ros: this.rosClient,
            name: this.nodeName + '/change_state',
            serviceType: 'lifecycle_msgs/srv/ChangeState'
        });

        this.getAvailableLilfecycleTransitionsService = new RosLib.Service({
            ros: this.rosClient,
            name: this.nodeName + '/get_available_transitions',
            serviceType: 'lifecycle_msgs/srv/GetAvailableTransitions'
        });

        this.getLifecycleStateService = new RosLib.Service({
            ros: this.rosClient,
            name: this.nodeName + '/get_state',
            serviceType: 'lifecycle_msgs/srv/GetState'
        });
    }

    createSubscriptions = () => {
        const rosoutTopic = new RosLib.Topic({
            ros: this.rosClient,
            name: 'rosout',
            messageType : 'rcl_interfaces/msg/Log'
          });

        rosoutTopic.subscribe(message => {
            dispatcher.dispatch({
                type: 'ROSOUT_MSG',
                rosout: message
            });
        });
    }

    refreshState = () =>
    {
        this.refreshLifecycleState();
    }

    refreshLifecycleState = () => {
        this.callGetAvailableLifecycleTransitions();
        this.callGetLifecycleStateService();
    }

    callGetLifecycleStateService = () =>
    {
        const request = new RosLib.ServiceRequest({});
        this.getLifecycleStateService.callService(request, response => {
            dispatcher.dispatch({
                type: 'LIFECYCLE_STATE',
                currentState: response.current_state
            });
        });
    }

    callLifecycleChangeStateService = (transition) => {
        const request = new RosLib.ServiceRequest({
            transition: {
                id: '',
                label: transition
            }
        });

        this.changeLifecycleChaneStateService.callService(request, result => {
            this.refreshLifecycleState();
        })
    }

    callGetAvailableLifecycleTransitions = () => {
        const request = new RosLib.ServiceRequest({});
        this.getAvailableLilfecycleTransitionsService.callService(request, result => {
            dispatcher.dispatch({
                type: 'LIFECYCLE_AVAILABLE_TRANSITIONS',
                availableTransitions: result.available_transitions
            });
        })
    }

    clearRosoutMessages = () => {
        dispatcher.dispatch({
            type: 'CLEAR_ROSOUT_MSGS'
        });
    }

}

const commonActions = new CommonActions();

export default commonActions;