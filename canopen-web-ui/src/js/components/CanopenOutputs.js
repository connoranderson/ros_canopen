import React from 'react';
import Title from './Title';
import { withStyles } from '@material-ui/styles';
import FormLabel from '@material-ui/core/FormLabel';
import FormControl from '@material-ui/core/FormControl';
import FormGroup from '@material-ui/core/FormGroup';
import FormControlLabel from '@material-ui/core/FormControlLabel';
import Switch from '@material-ui/core/Switch';
import Button from '@material-ui/core/Button';

import CommonStore from '../stores/CommonStore';
import commonActions from '../actions/CommonActions';

const styles = theme => ({

});

class CanopenOutputs extends React.Component {
    constructor(...args) {
        super(...args);

        this.state = {
            canopenOutputs: CommonStore.getState().get('canopenOutputs')
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
            canopenOutputs: CommonStore.getState().get('canopenOutputs')
        });
    }

    render() {
        const canopenNode = 'node_1';
        const nodeOutputs = this.state.canopenOutputs.get(canopenNode);
        const digitalOutputRows = [];
        if (typeof nodeOutputs !== 'undefined') {
            nodeOutputs.get('digitalOutputNames').forEach((outputName, index) => {
                const physicalPort = "DO" + index.toString();
                const output_on = nodeOutputs.getIn(['digitalOutputs', index]);
                digitalOutputRows.push(
                    <FormControlLabel
                        control={
                        <Switch 
                            checked={output_on} 
                            color='primary'
                            onChange={ (event) => commonActions.setDigitalCanopenOutput(canopenNode, index, outputName, event.target.checked)}
                        /> }
                        label={physicalPort + ": " + outputName}
                    />
                )
            });
        }

        return (
            <React.Fragment>
                <Title>CANopen Outputs</Title>
                <Button onClick={() => commonActions.testCall()}>Call</Button>
                <FormControl component="fieldset">
                    <FormLabel component="legend">Digital outputs</FormLabel>
                    <FormGroup>
                        {digitalOutputRows}
                    </FormGroup>
                </FormControl>
            </React.Fragment>
        );
    }
}

export default withStyles(styles)(CanopenOutputs);