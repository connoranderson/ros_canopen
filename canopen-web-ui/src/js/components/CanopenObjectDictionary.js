
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
import Switch from '@material-ui/core/Switch';
import FormControlLabel from '@material-ui/core/FormControlLabel';

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
      selectedNode: '',
      cached: false
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
    CommonActions.refreshObjectDictionaries([event.target.value]);
  };

  handleToDeviceChange = event => {
    this.setState({
      cached: event.target.checked
    });
  }

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

    const data_type_names = [
      'Unknown',
      'Unknown',
      'Int8',
      'Int16',
      'Int32',
      'UInt8',
      'UInt16',
      'UInt32',
      'Real32',
      'VisibleString',
      'OctetString',
      'UnicodeString',
      'Unknow',
      'Unknown',
      'Unknown',
      'Domain',
      'Real64',
      'Unknow',
      'Unknown',
      'Unknow',
      'Unknown',
      'Int64',
      'Unknow',
      'Unknown',
      'Unknow',
      'Unknown',
      'Unknown',
      'UInt64',
    ]

    canopenDictionaryEntries.forEach( entry => {
      entry.data_type_name = data_type_names[entry.data_type]
    });

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
        <FormControlLabel
          control={
            <Switch
              checked={this.state.cached}
              onChange={this.handleToDeviceChange}
            />
          }
          label='Cached'
        />
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
            { title: 'Index', field: 'index', editable: 'never'},
            { title: 'Parameter Name', field: 'parameter_name', editable: 'never'},
            { title: 'Data Type', field: 'data_type_name', editable: 'never'},
            { title: 'Value', field: 'value' },
          ]}
          data={canopenDictionaryEntries}
          title={selectedNode}
          editable={{
            onRowUpdate: (newData, oldData) => 
              new Promise(resolve => {
                CommonActions.updateCanopenObject(newData, oldData, selectedNode);
                resolve();
              })}}
          actions={[
            rowData => ({
              icon: 'refresh',
              tooltip: 'Read value',
              onClick: (event, rowData) => CommonActions.callCanopenGetObjectService(selectedNode, rowData, this.state.cached)
            })
          ]}
        />
      </React.Fragment>
    );
  }
}

export default withStyles(styles)(Rosparams);