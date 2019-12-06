import React from 'react';
import { withStyles } from '@material-ui/core/styles';
import Title from './Title';
import Grid from '@material-ui/core/Grid'

import CommonStore from '../stores/CommonStore';
import { ExpansionPanel, ExpansionPanelSummary, ExpansionPanelDetails} from '@material-ui/core';
import Typography from '@material-ui/core/Typography';

import { CheckCircle, Warning, Error, HourglassEmpty, ExpandMore } from '@material-ui/icons';
import { lightBlue, orange, pink, deepPurple } from '@material-ui/core/colors';
import { Table, TableHead, TableCell, TableBody, TableRow} from '@material-ui/core';

const styles = theme => ({
    diagnosticItemSummary: {
        fontSize: theme.typography.unit * 2,
        marginLeft: 10
    }
});

class RuntimeMonitor extends React.Component {
    constructor(...args) {
        super(...args);

        this.state = {
            diagnosticItems: CommonStore.getState().get('diagnosticItems')
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
            diagnosticItems: CommonStore.getState().get('diagnosticItems')
        })
    }

    render() {
        const { classes } = this.props;
        const sortedDiagnosticItems = this.state.diagnosticItems
            .sort((a, b) => a.get('name').localeCompare(b.get('name')));

        const diagnosticsPanelList = [];

        sortedDiagnosticItems.forEach(diagnosticItem => {
            diagnosticsPanelList.push(
                <DiagnosticExpansionPanel
                    diagnosticItem={diagnosticItem}
                    classes={classes}
                />);
        });

        return (
            <React.Fragment>
                <Title>RuntimeMonitor</Title>
                {diagnosticsPanelList}
            </React.Fragment>
        )
    }
}

const DiagnosticExpansionPanel = (props) => {

    const {diagnosticItem, classes} = props;

    let icon;
    switch (diagnosticItem.get('level')) {
        case 0: {
            icon = <CheckCircle style={{ color: lightBlue.A200 }} />;
            break;
        }
        case 1: {
            icon = <Warning style={{ color: orange.A200 }} />;
            break;
        }
        case 2: {
            icon = <Error style={{ color: pink.A200 }} />;
            break;
        }
        case 3: {
            icon = <HourglassEmpty style={{ color: deepPurple.A200 }} />;
            break;
        }
        default:
        // Do nothing
    }

    const nameArray = diagnosticItem.get('name').split(': ');
    const noPrefixName = nameArray[nameArray.length - 1];

    return (
        <ExpansionPanel key={diagnosticItem.get('name')}>
            <ExpansionPanelSummary expandIcon={<ExpandMore/>}>
                <Grid container justify='flex-start'>
                    <Grid item>
                        {icon}
                    </Grid>
                    <Grid>
                        <Typography className={classes.diagnosticItemSummary}>
                            {noPrefixName}
                        </Typography>
                    </Grid>
                </Grid>
            </ExpansionPanelSummary>
            <ExpansionPanelDetails>
                <DiagnosticTable 
                    diagnosticItem={diagnosticItem} 
                    classes={classes}
                />
            </ExpansionPanelDetails>
        </ExpansionPanel>
    )
}

const DiagnosticTable = (props) => {
    const {diagnosticItem} = props

    const nameArray = diagnosticItem.get('name').split(': ');
    const sourceNode = nameArray[0];

    const tableRows = [];

    tableRows.push(
        <TableRow key='message'>
          <TableCell>{'Message'}</TableCell>
          <TableCell align="right">{diagnosticItem.get('message')}</TableCell>
        </TableRow>
      );

      tableRows.push(
        <TableRow key='hardwareId'>
          <TableCell>{'Hardware Id'}</TableCell>
          <TableCell align="right">{diagnosticItem.get('hardware_id')}</TableCell>
        </TableRow>
      );

      tableRows.push(
        <TableRow key='node'>
          <TableCell>Source Node</TableCell>
          <TableCell align="right">{sourceNode}</TableCell>
        </TableRow>
      );

      diagnosticItem.get('values').forEach(keyValuePair => {
        const value = keyValuePair.get('value');
        const key = keyValuePair.get('key');
        tableRows.push(
          <TableRow key={key}>
            <TableCell>{key}</TableCell>
            <TableCell align="right">{value}</TableCell>
          </TableRow>
        );
      });


    return (
        <Table>
            <TableHead>
                <TableCell>Name</TableCell>
                <TableCell align='right'>Value</TableCell>
            </TableHead>
            <TableBody>
                {tableRows}
            </TableBody>
        </Table>
    )
}

export default withStyles(styles)(RuntimeMonitor);