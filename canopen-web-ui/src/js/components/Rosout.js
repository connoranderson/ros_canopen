import React from 'react';
import { withStyles } from '@material-ui/core/styles';
import Title from './Title';
import CommonStore from '../stores/CommonStore';
import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import { ListItemText } from '@material-ui/core';
import ListItemIcon from '@material-ui/core/ListItemIcon';
import InfoIcon from '@material-ui/icons/Info';
import WarningIcon from '@material-ui/icons/Warning';
import ErrorIcon from '@material-ui/icons/Error';
import ClearAll from '@material-ui/icons/ClearAll';
import Grid from '@material-ui/core/Grid';
import IconButton from "@material-ui/core/IconButton";
import CommonActions from '../actions/CommonActions';

const styles = theme => ({
    ul: {
        padding: 0,
    }
});


class Rosout extends React.Component {
    constructor(...args) {
        super(...args);

        this.state = {
            rosoutMsgs: CommonStore.getState().get('rosoutMsgs')
        }
    }

    componentDidMount() {
        CommonStore.on('change', this.storeChange);
    }

    componentWillUnmount() {
        CommonStore.removeListener('change', this.storeChange);
    }

    storeChange = () => {
        this.setState({
            rosoutMsgs: CommonStore.getState().get('rosoutMsgs')
        });
    }

    render() {
        const {classes} = this.props;
        const rosoutListItems = []

        this.state.rosoutMsgs.forEach(rosout_entry => {
            const stamp = rosout_entry.getIn(['stamp', 'sec']) + rosout_entry.getIn(['stamp', 'nanosec']);
            const level = rosout_entry.get('level');

            let icon;
            icon = <InfoIcon color='primary' />;
            if (level === 30) {
                icon = <WarningIcon color='secondary' />;
            } else if (level === 40) {
                icon = <ErrorIcon color='error' />;
            }

            rosoutListItems.push(
                <ListItem key={stamp}>
                    <ListItemIcon>
                        {icon}
                    </ListItemIcon>
                    <ListItemText>
                        {rosout_entry.get('msg')}
                    </ListItemText>
                </ListItem>
            );
        });

        return (
            <React.Fragment>
                <Grid container justify='space-between'>
                    <Grid item>
                        <Title>ROS Console</Title>
                    </Grid>
                    <Grid item>
                        <IconButton
                            onClick={() => CommonActions.clearRosoutMessages()}
                        >
                            <ClearAll color='primary'/>
                        </IconButton>
                    </Grid>
                </Grid>
                <List>
                    <ul className={classes.ul}>
                        {rosoutListItems}
                    </ul>
                </List>
            </React.Fragment>
        )
    }
}

export default withStyles(styles)(Rosout);