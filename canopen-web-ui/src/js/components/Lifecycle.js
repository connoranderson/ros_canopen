/* eslint-disable no-script-url */
import React from 'react';
import { withStyles } from '@material-ui/core/styles';
import Button from '@material-ui/core/Button'
import Title from './Title';
import Grid from '@material-ui/core/Grid'

import CommonActions from '../actions/CommonActions';
import CommonStore from '../stores/CommonStore';
import { IconButton, Typography } from '@material-ui/core';
import RefreshIcon from '@material-ui/icons/Refresh';

const styles = theme => ({
  depositContext: {
    flex: 1,
  },
  button: {
    margin: theme.spacing(1),
  },
});

class Lifecycle extends React.Component {
  constructor(...args) {
    super(...args);

    this.state = {
      availableLifecycleTransitions: CommonStore.getState().get('availableLifecycleTransitions'),
      lifecycleState: CommonStore.getState().get('lifecycleState')
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
      availableLifecycleTransitions: CommonStore.getState().get('availableLifecycleTransitions'),
      lifecycleState: CommonStore.getState().get('lifecycleState')
    })
  }

  render() {
    const { classes } = this.props;
    const { availableLifecycleTransitions } = this.state;

    const transitionButtons = [];

    availableLifecycleTransitions.forEach(transitionName => {
      transitionButtons.push(
        <Button
          variant="contained"
          color="primary"
          className={classes.button}
          key={transitionName}
          onClick={() => CommonActions.callLifecycleChangeStateService(transitionName)}
        >
          {transitionName}
        </Button>
      )
    });

    return (
      <React.Fragment>
        <Grid container justify='space-between'>
        <Grid item>
          <Title>Lifecycle</Title>
        </Grid>
        <Grid item>
          <IconButton
            onClick={() => CommonActions.callGetAvailableLifecycleTransitions()}
          >
            <RefreshIcon/>
          </IconButton>
        </Grid>
        </Grid>
        <Typography>
          Current state: {this.state.lifecycleState}
        </Typography>
        <div>
          {transitionButtons}
        </div>
      </React.Fragment>
    );
  }
}

export default withStyles(styles)(Lifecycle);
