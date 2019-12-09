import dispatcher from '../dispatcher';

import RosLib from 'roslib';

// TODO(sam): figure out how to refactor/split up this...

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

        this.createServiceClients();
        this.createSubscriptions();
    }

    createServiceClients = () => {
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

        this.listParametersService = new RosLib.Service({
            ros: this.rosClient,
            name: this.nodeName + '/list_parameters',
            serviceType: 'rcl_interfaces/srv/ListParameters'
        });

        this.getParametersService = new RosLib.Service({
            ros: this.rosClient,
            name: this.nodeName + '/get_parameters',
            serviceType: 'rcl_interfaces/srv/GetParameters'
        });

        this.setParametersService = new RosLib.Service({
            ros: this.rosClient,
            name: this.nodeName + '/set_parameters',
            serviceType: 'rcl_interfaces/srv/SetParameters'
        });

        this.listObjectDictionariesService = new RosLib.Service({
            ros: this.rosClient,
            name: '/list_object_dictionaries',
            serviceType: 'canopen_msgs/srv/ListObjectDictionaries'
        });

        this.canopenGetObjectService = new RosLib.Service({
            ros: this.rosClient,
            name: '/get_object',
            serviceType: 'canopen_msgs/srv/GetObject'
        });

        this.canopenSetObjectService = new RosLib.Service({
            ros: this.rosClient,
            name: '/set_object',
            serviceType: 'canopen_msgs/srv/SetObject'
        });
    }

    createSubscriptions = () => {
        const rosoutTopic = new RosLib.Topic({
            ros: this.rosClient,
            name: 'rosout',
            messageType: 'rcl_interfaces/msg/Log'
        });

        rosoutTopic.subscribe(message => {
            dispatcher.dispatch({
                type: 'ROSOUT_MSG',
                rosout: message
            });
        });

        this.diagnosticsListener = new RosLib.Topic({
            ros: this.rosClient,
            name: '/diagnostics',
            messageType: 'diagnostic_msgs/DiagnosticArray'
        });

        this.diagnosticsListener.subscribe(message => {
            dispatcher.dispatch({
                type: 'DIAGNOSTIC_ITEMS',
                diagnosticItems: message.status
            });
        });

        this.lifecycleTransitionEvent = new RosLib.Topic({
            ros: this.rosClient,
            name: '/canopen_chain/transition_event',
            messageType: 'lifecycle_msgs/msg/TransitionEvent'
        });

        this.lifecycleTransitionEvent.subscribe(message => {
            this.callGetAvailableLifecycleTransitionsService();
            this.callGetLifecycleStateService();
        });

        this.canopenInputs = new RosLib.Topic({
            ros: this.rosClient,
            name: '/node_1/inputs',
            messageType: 'canopen_msgs/msg/DeviceInputs'
        });

        this.canopenInputs.subscribe(message => {
            dispatcher.dispatch({
                type: 'CANOPEN_INPUTS',
                node_name: 'node_1',
                digital_input_names: message.digital_input_names,
                digital_inputs: message.digital_inputs
            });
        });
    }

    refreshState = () => {
        this.refreshLifecycleState();
    }

    refreshLifecycleState = () => {
        this.callGetAvailableLifecycleTransitionsService();
        this.callGetLifecycleStateService();
        this.refreshParameters();
    }

    refreshParameters = () => {
        const request = new RosLib.ServiceRequest({
            prefixes: [],
            depth: 0
        });

        this.listParametersService.callService(request, response => {
            const { names } = response.result;

            const request = new RosLib.ServiceRequest({
                names
            });
            this.getParametersService.callService(request, response => {
                const { values } = response;
                dispatcher.dispatch({
                    type: 'PARAMETER_VALUES',
                    names,
                    values
                });
            });

        });
    }

    refreshObjectDictionaries = (nodeNames) =>
    {
        const request = new RosLib.ServiceRequest({
            nodes: nodeNames
        });
        this.listObjectDictionariesService.callService(request, response => {
            dispatcher.dispatch({
                type: 'CANOPEN_OBJECT_DICTIONARIES',
                object_dictionaries: response.object_dictionaries
            });
        })
    }

    callCanopenGetObjectService = (nodeName, canopenObject, cached) => {
        const request = new RosLib.ServiceRequest({
            node: nodeName,
            object: canopenObject.index,
            cached
        });

        this.canopenGetObjectService.callService(request, response => {
            if (response.success) {
                dispatcher.dispatch({
                    type: 'CANOPEN_OBJECT_VALUE',
                    nodeName: request.node,
                    objectIndex: request.object,
                    value: response.value
                });
            } else {
                // console.warn("Failed to get canopen object: " + response.message);
                alert("Failed to get canopen object: " + response.message);
            }
        });
    }

    callGetLifecycleStateService = () => {
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

        this.changeLifecycleChaneStateService.callService(request, response => {
            this.refreshLifecycleState();
        })
    }

    callGetAvailableLifecycleTransitionsService = () => {
        const request = new RosLib.ServiceRequest({});
        this.getAvailableLilfecycleTransitionsService.callService(request, response => {
            dispatcher.dispatch({
                type: 'LIFECYCLE_AVAILABLE_TRANSITIONS',
                availableTransitions: response.available_transitions
            });
        })
    }


    clearRosoutMessages = () => {
        dispatcher.dispatch({
            type: 'CLEAR_ROSOUT_MSGS'
        });
    }

    updateCanopenObject = (newData, oldData, nodeName) => {
        const request = new RosLib.ServiceRequest({
            node: nodeName,
            object: oldData.index,
            value: newData.value,
            cached: true
        });

        this.canopenSetObjectService.callService(request, response => {
            if (response.success) {
               this.callCanopenGetObjectService(nodeName, newData, true);
            } else {
                alert("Failed to set canopen object: " + response.message);
            }
        });
    }

    updateRosParameter = (newData, oldData) => {
        // TODO(sam): share these between action and store
        // const PARAMETER_NOT_SET = 0
        const PARAMETER_BOOL = 1
        const PARAMETER_INTEGER = 2
        const PARAMETER_DOUBLE = 3
        const PARAMETER_STRING = 4
        // const PARAMETER_BYTE_ARRAY = 5
        // const PARAMETER_BOOL_ARRAY = 6
        // const PARAMETER_INTEGER_ARRAY = 7
        // const PARAMETER_DOUBLE_ARRAY = 8
        const PARAMETER_STRING_ARRAY = 9

        if (oldData.name !== newData.name) {
            console.warn("Changing the name of ROS parameters is not supported yet!");
        } else {
            const { type } = newData;
            const valueString = newData.valueString;
            const parameterValue = { type };
            switch (type) {
                case PARAMETER_BOOL:
                    {
                        parameterValue.bool_value = (valueString === 'true');
                        break;
                    }
                case PARAMETER_INTEGER:
                    {
                        parameterValue.integer_value = parseInt(valueString);
                        break;
                    }
                case PARAMETER_DOUBLE:
                    {
                        parameterValue.double_value = parseFloat(valueString);
                        break;
                    }
                case PARAMETER_STRING:
                    {
                        parameterValue.string_value = valueString;
                        break;
                    }
                case PARAMETER_STRING_ARRAY:
                    {
                        parameterValue.string_array_value = JSON.parse(valueString);
                        break;
                    }
                default:
                    {
                        console.warn('Writing parameter type ' + newData.typeName + ' is not supported yet!');
                        return;
                    }
            }

            const request = new RosLib.ServiceRequest({
                parameters: [{
                    name: newData.name,
                    value: parameterValue
                }]
            });

            this.setParametersService.callService(request, response => {
                this.refreshParameters();
            });
        }

    }

}

const commonActions = new CommonActions();

export default commonActions;