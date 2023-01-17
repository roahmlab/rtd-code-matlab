RTD Code Mixin Classes
======================

Mixins are utility classes that you add as a superclass to a given class to quickly add
functionality. They do anything from provide additional generated properties for easy
names or identification, or add quick array functionality.

.. mat:autoclass:: utils.NamedClass
   :show-inheritance:
   :members:
   :undoc-members:

The OptionsClass mixin introduces an instanceOptions property which holds a struct of
all sorts of configuration options. It also includes utility code for the merging of
these structs and is designed to be used with MATLAB arguments for Name-Value argument
generation.

.. mat:autoclass:: utils.OptionsClass
   :show-inheritance:
   :members:
   :undoc-members:

.. mat:autoclass:: utils.UUIDbase
   :show-inheritance:
   :members:
   :undoc-members:
