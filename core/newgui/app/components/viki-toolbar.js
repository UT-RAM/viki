import Ember from 'ember';

export default Ember.Component.extend({
	actions: {
		connCheck(){
			vikiBackend.connCheck();
		}
	}
  // here should we create data bindings with the python
});
