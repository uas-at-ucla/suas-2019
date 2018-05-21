import React, { Component } from 'react';
import {
  SortableContainer,
  SortableElement,
  arrayMove
} from 'react-sortable-hoc';
import './MissionPlanner.css';

const oppositeOf = {'comeToStop': 'flyThrough'}

const SortableItem = SortableElement(({ command, changedCommands, myIndex, self }) => {
  let type = command.type;
  let fields = self.props.commandTypes[type];

  return (
    <tr
      className="command_row"
      onClick={event => self.onCommandClick(myIndex, event)}
    >
      <td><div>{myIndex + 1}</div></td>
      <td><div>{command.name}</div></td>
      <td><div>{type.replace('Command', '')}</div></td>
      {fields.map((field, index) => {
        return self.commandField(command, field.name, index);
      })}
    </tr>
  );
});

class MySortableItem extends SortableItem {
  shouldComponentUpdate(nextProps, nextState) {
    let result = nextProps.changedCommands === null || (
        nextProps.changedCommands.startIndex <= nextProps.myIndex &&
        nextProps.myIndex <= nextProps.changedCommands.endIndex);
    return result;
  }
}

const SortableList = SortableContainer(({ commands, changedCommands, self }) => {
  return (
    <div className="scrollbar">
      <table id="commandListOverview">
        <tbody>
          {commands.map((command, index) => (
            <MySortableItem
              key={index}
              index={index}
              command={command}
              changedCommands={changedCommands}
              myIndex={index}
              self={self}
            />
          ))}
        </tbody>
      </table>
    </div>
  );
});

class MissionPlannerOverview extends Component {
  render() {
    return (
      <SortableList
        commands={this.props.homeState.commands}
        changedCommands={this.props.homeState.changedCommands}
        onSortEnd={this.onSortEnd}
        self={this}
        transitionDuration={200}
        distance={2}
      />
    );
  }

  onSortEnd = ({ oldIndex, newIndex }) => {
    if (oldIndex !== newIndex) {
      this.props.setHomeState({
        commands: arrayMove(this.props.homeState.commands, oldIndex, newIndex),
        changedCommands: {
          startIndex: Math.min(oldIndex, newIndex), 
          endIndex: Math.max(oldIndex, newIndex)
        }
      });
    }
  };

  onCommandClick(index, event) {
    if (event.target.tagName === 'TD' || event.target.tagName === 'DIV') {
      this.props.setHomeState({ focusedCommand: index });
    }
  }

  commandField(command, field, id) {
    let type = command.type;
    let keys = field.split('.');
    let subcommand = command[type];
    let key = null;
    for (key of keys) {
      if (Array.isArray(subcommand)) {
        subcommand = subcommand[parseInt(key)];
      } else {
        type = this.props.commandTypes[type].find(el => el.name === key).type;
        subcommand = subcommand[key];
      }
      if (subcommand == null) {
        return null;
      }
    }

    if (typeof subcommand !== 'object') {
      if (type === 'bool') {
        return <td key={id}><div>{subcommand ? field : oppositeOf[field]}</div></td>;
      } else {
        return <td key={id}><div>{parseFloat(subcommand).toFixed(3)}</div></td>;
      }
    } else {
      let fields = this.props.commandTypes[type];
      if (Array.isArray(subcommand)) {
        return (
          <td key={id}>
            <div>
            <table>
              <tbody>
                {subcommand.map((cmd, i) => 
                  <tr key={i}>
                    {fields.map((next_field, j) => {
                      return this.commandField(command, field+'.'+i+'.'+next_field.name, j);
                    })}
                  </tr>
                )}
              </tbody>
            </table>
            </div>
          </td>
        );
      }
      return (
        <td key={id}>
          <div>
          <table>
            <tbody>
              <tr>
                {fields.map((next_field, i) => {
                  return this.commandField(command, field+'.'+next_field.name, i);
                })}
              </tr>
            </tbody>
          </table>
          </div>
        </td>
      );
    }
  }
}

export default MissionPlannerOverview;
