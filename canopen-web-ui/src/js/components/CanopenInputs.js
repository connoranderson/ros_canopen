import React from 'react';
import Title from './Title';
import { withStyles } from '@material-ui/styles';
import Table from '@material-ui/core/Table';
import TableBody from '@material-ui/core/TableBody';
import TableCell from '@material-ui/core/TableCell';
import TableHead from '@material-ui/core/TableHead';
import TableRow from '@material-ui/core/TableRow';

import CommonStore from '../stores/CommonStore';

const styles = theme => ({

});

class CanopenInputs extends React.Component {
    constructor(...args) {
        super(...args);

        this.state = {
            canopenInputs: CommonStore.getState().get('canopenInputs')
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
            canopenInputs: CommonStore.getState().get('canopenInputs')
        });
    }

    render() {
        const canopenNode = 'node_1';
        const nodeInputs = this.state.canopenInputs.get(canopenNode);
        const digitalInputRows = [];
        if (typeof nodeInputs !== 'undefined') {
            nodeInputs.get('digitalInputNames').forEach((inputName, index) => {
                digitalInputRows.push(
                    <TableRow>
                        <TableCell>{inputName}</TableCell>
                        <TableCell align="right">Digital Input</TableCell>
                        <TableCell align="right">{"DI" + index.toString()}</TableCell>
                        <TableCell align="right">{nodeInputs.getIn(['digitalInputs', index]).toString()}</TableCell>
                    </TableRow>
                )
            });
        }

        return (
            <React.Fragment>
                <Title>CANopen Inputs</Title>
                <Table aria-label="inputs table">
                    <TableHead>
                        <TableRow>
                            <TableCell>Input Name</TableCell>
                            <TableCell align="right">Type</TableCell>
                            <TableCell align="right">Physical Port</TableCell>
                            <TableCell align="right">Value</TableCell>
                        </TableRow>
                    </TableHead>
                    <TableBody>
                        {digitalInputRows}
                    </TableBody>
                </Table>
            </React.Fragment>
        );
    }
}

export default withStyles(styles)(CanopenInputs);