/************************************
SoftContact.h
Claire Lackner

This implements a class, SoftContact, creates a contact between two models
which is not rigid.  The contact is modelled by allowing the two surfaces 
to interpenetrate.  The models are locally fitted by second order polynomials
around the contact.  This polynomial surfaces are allowed to interpenetrate
and interact governed by Hertz contact theory.

*************************************/