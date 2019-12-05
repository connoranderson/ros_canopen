import {
  EventEmitter
} from 'events';
import dispatcher from '../dispatcher';

import Immutable from 'immutable';

class RuntimeMonitorStore extends EventEmitter {
  constructor() {
    super();

    this.state = Immutable.fromJS(RuntimeMonitorStore.defaultState);
  }

  getState() {
    return this.state;
  }

  handleActions(action) {
    switch (action.type) {
      case 'ROS_CLIENT':
        this.state = this.state.set('rosClient', action.rosClient);
        this.emit('rosClientChange');
        break;
      case 'DIAGNOSTIC_ITEMS':
        let diagnosticItemMap = {};
        action.diagnosticItems.forEach(diagnosticItem => {
          diagnosticItemMap[diagnosticItem.name] = Immutable.fromJS(diagnosticItem);
        });

        this.state = this.state.set(
          'diagnosticItems',
          this.state.get('diagnosticItems').merge(diagnosticItemMap)
        );

        this.emit('change');
        break;
      default:
        // Do nothing
    }
  }
}

RuntimeMonitorStore.defaultState = {
  diagnosticItems: {},
  rosClient: null,
  oldRosClient: null,
};

const runtimeMonitorStore = new RuntimeMonitorStore();
dispatcher.register(
  runtimeMonitorStore.handleActions.bind(runtimeMonitorStore)
);

export default runtimeMonitorStore;
