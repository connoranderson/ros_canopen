
/* eslint-disable no-script-url */
import React from 'react';
import { withStyles } from '@material-ui/core/styles';
import Title from './Title';
import Grid from '@material-ui/core/Grid'

import CommonActions from '../actions/CommonActions';
import CommonStore from '../stores/CommonStore';
import IconButton from '@material-ui/core/IconButton';
import RefreshIcon from '@material-ui/icons/Refresh';

import MaterialTable from 'material-table';

import FormControl from '@material-ui/core/FormControl';
import Select from '@material-ui/core/Select';
import MenuItem from '@material-ui/core/MenuItem';
import InputLabel from '@material-ui/core/InputLabel';

const styles = theme => ({
  formControl: {
    margin: theme.spacing(1),
    minWidth: 140,
  }
});

class Rosparams extends React.Component {
  constructor(...args) {
    super(...args);

    this.state = {
      rosParams: CommonStore.getState().get('rosParams'),
      canopenObjectDictionaries: CommonStore.getState().get('canopenObjectDictionaries'),
      selectedNode: ''
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
      rosParams: CommonStore.getState().get('rosParams'),
      canopenObjectDictionaries: CommonStore.getState().get('canopenObjectDictionaries')
    })
  }

  handleNodeSelectorChange = event => {
    this.setState({
      selectedNode: event.target.value
    });
  };

  render() {
    const { classes } = this.props;
    const { selectedNode } = this.state;

    let canopenNodes = [];

    this.state.rosParams.forEach(parameter => {
      if (parameter.get('name') === 'canopen_nodes') {
        canopenNodes = JSON.parse(parameter.get('valueString'))
      }
    });

    const nodeMenuItems = [];
    canopenNodes.forEach( nodeName => {
      nodeMenuItems.push(
        <MenuItem key={nodeName} value={nodeName}>
          {nodeName}
        </MenuItem>
      );
    });

    let canopenDictionaryEntries = [];
    if (this.state.canopenObjectDictionaries.get(selectedNode))
    {
      canopenDictionaryEntries = this.state.canopenObjectDictionaries.get('node_1').toJS()
    }

    return (
      <React.Fragment>
        <Grid container justify='space-between'>
        <Grid item>
          <Title>Object Dictionary</Title>
        </Grid>
        <Grid item>
          <FormControl className={classes.formControl}>
            <InputLabel>CANopen Node</InputLabel>
            <Select 
              value={selectedNode}
              onChange={this.handleNodeSelectorChange}  
            >
              {nodeMenuItems}
            </Select>
          </FormControl>
        </Grid>
        <Grid item>
          <IconButton
            onClick={() => CommonActions.refreshObjectDictionaries(canopenNodes)}
          >
            <RefreshIcon color='primary'/>
          </IconButton>
        </Grid>
        </Grid>
        <MaterialTable
          columns={[
            { title: "Index", field: "index", editable: 'never'},
            { title: "Parameter Name", field: "parameter_name", editable: 'never'},
            { title: "Value", field: "value" },
          ]}
          data={canopenDictionaryEntries}
          title={selectedNode}
          editable={{
            onRowUpdate: (newData, oldData) => 
              new Promise(resolve => {
                CommonActions.updateRosParameter(newData, oldData);
                resolve();
              })}}
          actions={[
            rowData => ({
              icon: 'get_app',
              tooltip: 'Read value from device',
              onClick: (event, rowData) => CommonActions.callCanopenGetObjectService(selectedNode, rowData, false)
            }),
            rowData => ({
              icon: 'refresh',
              tooltip: 'Read cached value',
              onClick: (event, rowData) => CommonActions.callCanopenGetObjectService(selectedNode, rowData, true)
            })
          ]}
        />
      </React.Fragment>
    );
  }
}

export default withStyles(styles)(Rosparams);