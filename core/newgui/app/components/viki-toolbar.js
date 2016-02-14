import Ember from 'ember';

export default Ember.Component.extend({
	actions: {
		connCheck(){
			VikiBackend.connCheck();
		}
	}
  // here should we create data bindings with the python
});
