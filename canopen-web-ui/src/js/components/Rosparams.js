
/* eslint-disable no-script-url */
import React from 'react';
import { withStyles } from '@material-ui/core/styles';
import Title from './Title';
import Grid from '@material-ui/core/Grid'

import CommonActions from '../actions/CommonActions';
import CommonStore from '../stores/CommonStore';
import { IconButton } from '@material-ui/core';
import RefreshIcon from '@material-ui/icons/Refresh';

import MaterialTable from 'material-table';

const styles = theme => ({
});

class Rosparams extends React.Component {
  constructor(...args) {
    super(...args);

    this.state = {
      rosParams: CommonStore.getState().get('rosParams')
    };
  }

  componentDidMount() {
    CommonStore.on('change', this.storeChange);
  }

  componentWillUnmount() {
    CommonStore.removeListener('change', this.storeChange);
  }

  storeChange = () => {
    this.setState({
      rosParams: CommonStore.getState().get('rosParams')
    })
  }

  render() {
    // const { classes } = this.props;

    return (
      <React.Fragment>
        <Grid container justify='space-between'>
        <Grid item>
          <Title>Parameters</Title>
        </Grid>
        <Grid item>
          <IconButton
            onClick={() => CommonActions.refreshParameters()}
          >
            <RefreshIcon color='primary'/>
          </IconButton>
        </Grid>
        </Grid>
        <MaterialTable
          columns={[
            { title: "Name", field: "name" , editable: 'never'},
            { title: "Value", field: "valueString" },
          ]}
          data={this.state.rosParams.toJS()}
          title="canopen_chain_node"
          editable={{
            onRowUpdate: (newData, oldData) => 
              new Promise(resolve => {
                CommonActions.updateRosParameter(newData, oldData);
                resolve();
              })}}
        />
      </React.Fragment>
    );
  }
}

export default withStyles(styles)(Rosparams);