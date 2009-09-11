///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "topic_display_generated.h"

///////////////////////////////////////////////////////////////////////////
using namespace rxtools;

GenTopicDisplay::GenTopicDisplay( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );
	
	topic_tree_ = new wxTreeCtrl( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTR_DEFAULT_STYLE|wxTR_HIDE_ROOT|wxTR_MULTIPLE );
	bSizer1->Add( topic_tree_, 1, wxALL|wxEXPAND, 5 );
	
	this->SetSizer( bSizer1 );
	this->Layout();
	
	// Connect Events
	topic_tree_->Connect( wxEVT_COMMAND_TREE_ITEM_ACTIVATED, wxTreeEventHandler( GenTopicDisplay::onItemActivated ), NULL, this );
	topic_tree_->Connect( wxEVT_COMMAND_TREE_SEL_CHANGED, wxTreeEventHandler( GenTopicDisplay::checkIsTopic ), NULL, this );
	topic_tree_->Connect( wxEVT_COMMAND_TREE_SEL_CHANGING, wxTreeEventHandler( GenTopicDisplay::checkIsTopic ), NULL, this );
}

GenTopicDisplay::~GenTopicDisplay()
{
	// Disconnect Events
	topic_tree_->Disconnect( wxEVT_COMMAND_TREE_ITEM_ACTIVATED, wxTreeEventHandler( GenTopicDisplay::onItemActivated ), NULL, this );
	topic_tree_->Disconnect( wxEVT_COMMAND_TREE_SEL_CHANGED, wxTreeEventHandler( GenTopicDisplay::checkIsTopic ), NULL, this );
	topic_tree_->Disconnect( wxEVT_COMMAND_TREE_SEL_CHANGING, wxTreeEventHandler( GenTopicDisplay::checkIsTopic ), NULL, this );
}

GenTopicDisplayDialog::GenTopicDisplayDialog( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxVERTICAL );
	
	tree_panel_ = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	bSizer3->Add( tree_panel_, 1, wxEXPAND | wxALL, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxHORIZONTAL );
	
	ok_ = new wxButton( this, wxID_ANY, wxT("OK"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer4->Add( ok_, 0, wxALL, 5 );
	
	cancel_ = new wxButton( this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer4->Add( cancel_, 0, wxALL, 5 );
	
	bSizer3->Add( bSizer4, 0, wxALIGN_RIGHT, 5 );
	
	this->SetSizer( bSizer3 );
	this->Layout();
	
	// Connect Events
	ok_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTopicDisplayDialog::onOK ), NULL, this );
	cancel_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTopicDisplayDialog::onCancel ), NULL, this );
}

GenTopicDisplayDialog::~GenTopicDisplayDialog()
{
	// Disconnect Events
	ok_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTopicDisplayDialog::onOK ), NULL, this );
	cancel_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTopicDisplayDialog::onCancel ), NULL, this );
}
