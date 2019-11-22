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
            default:
                console.log(`action.type '${action.type}' not recognized!`);
        }
    }
}

CommonStore.defaultState = {
    availableLifecycleTransitions: [],
    lifecycleState: 'not available'
}

const commonStore = new CommonStore();
dispatcher.register(
    commonStore.handleActions.bind(commonStore)
);
export default commonStore;