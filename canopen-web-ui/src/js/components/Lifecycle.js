/* eslint-disable no-script-url */
import React from 'react';
import { makeStyles, ThemeProvider } from '@material-ui/core/styles';
import Button from '@material-ui/core/Button'
import Title from './Title';

import CommonActions from '../actions/CommonActions';

const useStyles = makeStyles(theme => ({
  depositContext: {
    flex: 1,
  },
  button: {
    margin: theme.spacing(1),
  },
}));

export default function Lifecycle() {
  const classes = useStyles();

  const transitionButtons = [];

  const transitions = ['create', 'configure', 'activate']
  transitions.forEach( transitionName => {
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
      <Title>Lifecycle</Title>
      <div>
        {transitionButtons}
      </div>
    </React.Fragment>
  );
}
