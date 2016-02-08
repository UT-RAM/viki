import Ember from 'ember';

export default Ember.Component.extend({
  module: {},
  canvasState: Ember.inject.service(),

    dragStart(event) {
      event.dataTransfer.setData("id", this.get('module').id);
      event.dataTransfer.setPosition("100px", "200px");
    }
});
