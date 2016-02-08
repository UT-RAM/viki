import Ember from 'ember';

export default Ember.Component.extend({
    canvasState: Ember.inject.service(),

    dragEnter(event) {
      event.preventDefault();
      console.log("dragenter");
    },
    drop(event) {
      event.preventDefault();
      this.get('canvasState').appendCurrentDraggingModule();
      console.log("hai!");
      console.log(event);
      //this.get('canvasState').app
    }
});
