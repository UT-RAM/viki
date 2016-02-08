import Ember from 'ember';

export default Ember.Service.extend({
    history: [],
    future: [],

    modules_in_canvas: [],
    current_dragging_module: null,

    addModuleToCanvas(module) {
      this.get('modules_in_canvas').addObject(module);
    },

    appendCurrentDraggingModule() {
      if (current_dragging_module != null) {
        addModuleToCanvas(this.get('current_dragging_module'));
        this.set('current_dragging_module', null);
      }
    }
});
