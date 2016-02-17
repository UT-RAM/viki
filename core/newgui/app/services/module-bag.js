import Ember from 'ember';

export default Ember.Service.extend({

  modules: [],

  init() {
    this.set('modules', [{
      'id': 'test_module',
      type: 'controller',
      'meta': {
        'name': "Test Module",
        'description': "This is a test module"
      },
      inputs: [{name: 'pos_in', message_type: 'position'}],
      outputs: [{name: 'pos_out', message_type: 'position'}]
    }]);
  },

  getModuleById(Id) {
      var mods = this.get('modules');
      for (var i=0; i <mods.length; i++) {
          if (mods[i].id == Id) {
              return $.extend(true, {}, mods[i]);
          }
      }
  }
});
