import { EventEmitter } from 'events';
import dispatcher from '../dispatcher';
import Immutable from 'immutable';

class CommonStore extends EventEmitter {
    constructor() {
        super();

        this.state = Immutable.fromJS(CommonStore.defaultState);
    }

    getState = () => {
        return this.state;
    }

    handleActions = (action) => {
        switch (action.type) {
            case 'ROS_CONNECTION_STATUS':
                {
                    break;
                }
            case 'LIFECYCLE_AVAILABLE_TRANSITIONS':
                {
                    const availableTransitions = []
                    action.availableTransitions.forEach(availableTransition => {
                        availableTransitions.push(availableTransition.transition.label);
                    });

                    this.state = this.state.set('availableLifecycleTransitions',
                        Immutable.fromJS(availableTransitions));

                    this.emit('change');
                    break;
                }
            case 'LIFECYCLE_STATE':
                {
                    this.state = this.state.set('lifecycleState', action.currentState.label);
                    this.emit('change');
                    break;
                }
            case 'ROSOUT_MSG':
                {
                    if (action.rosout.name === 'canopen_chain') {
                        const rosout_entry = {
                            msg: action.rosout.msg,
                            level: action.rosout.level,
                            stamp: {
                                sec: action.rosout.stamp.sec,
                                nanosec: action.rosout.stamp.nanosec,
                            }
                        };

                        this.state = this.state.update('rosoutMsgs',
                            arr => arr.push(Immutable.fromJS(rosout_entry)));

                        this.emit('change');
                    }

                    break;
                }
            case 'CLEAR_ROSOUT_MSGS':
                {
                    this.state = this.state.set('rosoutMsgs', Immutable.List());

                    this.emit('change');
                    break;
                }
            case 'PARAMETER_VALUES':
                {
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

                    const rosParams = [];
                    action.names.forEach((name, index) => {
                        const paramValue = action.values[index];
                        const { type } = paramValue;

                        let typeName = 'not supported';
                        let valueString = 'not supported';
                        switch (type) {
                            case PARAMETER_BOOL:
                                {
                                    typeName = 'bool';
                                    valueString = paramValue.bool_value.toString();
                                    break;
                                }
                            case PARAMETER_INTEGER:
                                {
                                    typeName = 'integer';
                                    valueString = paramValue.integer_value.toString();
                                    break;
                                }
                            case PARAMETER_DOUBLE:
                                {
                                    typeName = 'double';
                                    valueString = paramValue.double_value.toString();
                                    break;
                                }
                            case PARAMETER_STRING:
                                {
                                    typeName = 'string';
                                    valueString = paramValue.string_value.toString();
                                    break;
                                }
                            case PARAMETER_STRING_ARRAY:
                                {
                                    typeName = 'stringArray';
                                    valueString = JSON.stringify(paramValue.string_array_value);
                                    break;
                                }
                            default:
                                {
                                    console.log('Parameter of type ' + type.toString() + ' is not supported!');
                                }
                        }

                        rosParams.push({
                            name,
                            type,
                            typeName,
                            valueString
                        });
                    });

                    this.state = this.state.set('rosParams', Immutable.fromJS(rosParams));
                    this.emit('change');
                    break;
                }
            case 'CANOPEN_OBJECT_DICTIONARIES':
                {
                    action.object_dictionaries.forEach( dictionary => {
                        this.state = this.state.setIn(['canopenObjectDictionaries', dictionary.node], Immutable.fromJS(dictionary.object_descriptions));
                    })

                    this.emit('change');
                    break;
                }
            default:
                console.log(`action.type '${action.type}' not recognized!`);
        }
    }
}

CommonStore.defaultState = {
    availableLifecycleTransitions: [],
    lifecycleState: 'not available',
    rosoutMsgs: [],
    rosParams: [],
    canopenObjectDictionaries: {}
}

const commonStore = new CommonStore();
dispatcher.register(
    commonStore.handleActions.bind(commonStore)
);
export default commonStore;