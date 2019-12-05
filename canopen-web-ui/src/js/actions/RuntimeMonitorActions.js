// @flow

import dispatcher from '../dispatcher';

import RosLib from 'roslib';

class RuntimeMonitorActions {
  connect = rosClient => {

    this.diagnosticsListener = new RosLib.Topic({
      ros: rosClient,
      name: '/diagnostics',
      messageType: 'diagnostic_msgs/DiagnosticArray'
    });

    this.diagnosticsListener.subscribe(message => {
      dispatcher.dispatch({
        type: 'DIAGNOSTIC_ITEMS',
        diagnosticItems: message.status
      });
    });

  };

  disconnect = () => {
    this.diagnosticsListener.unsubscribe();
  };
}

const runtimeMonitorActions = new RuntimeMonitorActions();

export default runtimeMonitorActions;
